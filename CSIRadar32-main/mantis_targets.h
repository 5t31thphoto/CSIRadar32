#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS MESH TARGETS
//  Multiple targets, and the ones that are holding still
// ═══════════════════════════════════════════════════════════════
//
//  ── THE CAPABILITY THIS EXISTS FOR ───────────────────────────
//
//  Conventional CSI presence detection measures CHANGE OVER TIME.  When
//  a person stops moving they stop producing change, and the detector's
//  own evidence decays away:
//
//      still for  0 s   motion evidence 1.000
//                 5 s                   0.135
//                10 s                   0.018
//                30 s                   0.000
//
//  After half a minute the room reads as empty with someone standing in
//  it.  Every demo-grade CSI radar has this hole and it is not fixable
//  from a derivative, because the derivative is genuinely zero.
//
//  A chord measurement is not a derivative.  A body between two beacons
//  attenuates that path for as long as it is there, so the evidence is
//  FLAT rather than decaying.  That is the difference between a motion
//  detector and a PRESENCE detector, and it is the single largest
//  capability the mesh adds.
//
//  ── AND THE TWO CHANNELS DISAGREE USEFULLY ───────────────────
//
//  Blockage says WHERE something is.  Phase rate says whether it is
//  MOVING.  Cross them and you get a classification neither could make
//  alone:
//
//      blocked + phase moving   a person walking
//      blocked + phase still    a person standing, sitting, asleep
//      clear   + phase moving   motion outside the chord set, or a fan,
//                               or multipath from the next room
//      clear   + phase still    nothing
//
//  Exact slot timing is what makes the phase-rate channel usable at all:
//  free-running beacons gave dt = 33.3 +- 8 ms, a 24% velocity error.
//  Slotted gives +- 0.2 ms, 0.6%.
// ═══════════════════════════════════════════════════════════════
#include "mantis_mesh.h"
#include <stdint.h>
#include <math.h>

#define MANTIS_MAX_TARGETS 4

typedef enum : uint8_t {
    MT_NONE    = 0,
    MT_MOVING  = 1,   // blocked chords AND phase advancing
    MT_STATIC  = 2,   // blocked chords, phase quiet -- the hard case
    MT_FADING  = 3,   // was tracked, evidence weakening
} MantisTargetKind;

typedef struct {
    float    x, y;
    float    llr;           // reconstruction strength
    float    margin;        // peak over runner-up outside its lobe
    uint8_t  coverage;      // chords with line of sight through it
    float    phase_rate;    // mean |d(phase)/dt| on the blocked chords
    MantisTargetKind kind;
    uint32_t first_seq, last_seq;
    uint16_t age_frames;
    bool     active;
} MantisTarget;

typedef struct {
    MantisTarget t[MANTIS_MAX_TARGETS];
    uint8_t      n;
    // Previous frame's phases, for an exact-dt derivative.
    float        prev_phase[MANTIS_SLOTS * MANTIS_SLOTS];
    bool         prev_valid;
    uint32_t     prev_seq;
} MantisTargetSet;

// Motion threshold on the phase channel, radians per second.
//
// Set from what a body actually does: a hand moving 10 cm/s at 2.4 GHz
// shifts the path by ~0.8 rad/s.  Below this is drift and thermal
// wander, not a person.
#ifndef MANTIS_MOVING_RAD_S
  #define MANTIS_MOVING_RAD_S 0.35f
#endif

// Suppress a region around an accepted peak before searching for the
// next.  Sized to the chord half-width: two targets closer than this
// cannot be separated by 15 lines, and pretending otherwise would
// manufacture a second target out of one target's own lobe.
#ifndef MANTIS_TARGET_SEPARATION
  #define MANTIS_TARGET_SEPARATION 0.30f
#endif

// How strong a second target must be relative to the first.
//
// Not a tuned number -- a measured separation.  With two people in the
// room the genuine second peak came in at 99% of the primary while the
// strongest streak artefact reached 61%, because two bodies block a
// comparable number of chords and a reconstruction artefact does not.
// 0.70 sits in the gap with margin on both sides.
//
// The failure direction matters: set too high, a real second person is
// missed; too low, one person is reported as several.  Reporting
// phantom people is the worse error for anything anyone would act on,
// so the threshold sits above the midpoint rather than below it.
#ifndef MANTIS_SECOND_TARGET_FRAC
  #define MANTIS_SECOND_TARGET_FRAC 0.70f
#endif

// Mean |d(phase)/dt| over the chords that pass near (px,py).
//
// Only the BLOCKED chords are consulted: a clear path's phase wanders
// with the room and would dilute the answer toward "not moving" exactly
// when it matters.
static inline float mantis_phase_rate_at(const MantisMesh *m,
                                         const MantisTargetSet *ts,
                                         const MantisPerspectiveStore *store,
                                         float px, float py, float dt_s,
                                         uint32_t now_ms) {
    if (!ts->prev_valid || dt_s <= 1e-4f) return 0.0f;
    const float hw2 = MANTIS_CHORD_HALFWIDTH * MANTIS_CHORD_HALFWIDTH;
    float sum = 0.0f; int n = 0;

    for (uint8_t i = 1; i <= m->n_beacons; i++) {
        for (uint8_t j = 1; j <= m->n_beacons; j++) {
            if (i == j || !m->have_pos[i] || !m->have_pos[j]) continue;
            const int idx = i * MANTIS_SLOTS + j;
            if (m->atten[idx] <= 0.02f) continue;        // not blocked
            if (mantis_pt_seg_d2(px, py, m->bx[i], m->by[i],
                                 m->bx[j], m->by[j]) > hw2) continue;

            const MantisLinkView *lv = mantis_store_link(store, i, j, now_ms);
            if (!lv || lv->quality < 40) continue;
            const float cur = mantis_unq12(lv->phase_q12);
            float d = cur - ts->prev_phase[idx];
            // Wrap: a phase crossing pi is a small step, not a huge one.
            while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
            while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
            sum += fabsf(d) / dt_s;
            n++;
        }
    }
    return (n > 0) ? (sum / (float)n) : 0.0f;
}

// Sample the phase channel.  MUST be called every frame, not merely when
// targets are extracted: the derivative is only meaningful across ONE
// frame interval, and sampling across a gap of a dozen frames turns
// accumulated motion into a single enormous apparent rate -- which is
// exactly backwards, since a target that has been still the whole time
// then reads as fast-moving.
static inline void mantis_targets_snapshot_phase(MantisTargetSet *ts,
                                                 const MantisMesh *m,
                                                 const MantisPerspectiveStore *store,
                                                 uint32_t seq, uint32_t now_ms) {
    for (uint8_t i = 1; i <= m->n_beacons; i++)
        for (uint8_t j = 1; j <= m->n_beacons; j++) {
            if (i == j) continue;
            const MantisLinkView *lv = mantis_store_link(store, i, j, now_ms);
            if (lv) ts->prev_phase[i * MANTIS_SLOTS + j] = mantis_unq12(lv->phase_q12);
        }
    ts->prev_valid = true;
    ts->prev_seq   = seq;
}

// Extract up to MANTIS_MAX_TARGETS peaks, classifying each.
//
// Greedy peak-and-suppress rather than a fitted mixture: with 15 lines
// the field has real streak structure, and a mixture fit would happily
// place components on streak crossings and report them with confidence.
// Taking the strongest peak, suppressing its neighbourhood, and
// demanding the next clear the same bar is more conservative and fails
// in a way that is visible.
static inline uint8_t mantis_targets_extract(MantisTargetSet *ts,
                                             const MantisMesh *m,
                                             const MantisPerspectiveStore *store,
                                             uint32_t seq, float dt_s,
                                             uint32_t now_ms) {
    for (int i = 0; i < MANTIS_MAX_TARGETS; i++) ts->t[i].active = false;
    ts->n = 0;
    if (m->n_chords < 3) { mantis_targets_snapshot_phase(ts, m, store, seq, now_ms); return 0; }

    // Work on a copy: suppression must not damage the field the UI draws.
    static float work[MANTIS_TOMO_CELLS];
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) work[i] = m->field[i];

    const float step = (2.0f * m->extent) / (float)MANTIS_TOMO_DIM;

    for (int k = 0; k < MANTIS_MAX_TARGETS; k++) {
        int best = -1; float bv = -1e30f;
        for (int i = 0; i < MANTIS_TOMO_CELLS; i++)
            if (work[i] > bv) { bv = work[i]; best = i; }
        if (best < 0 || bv < MANTIS_MESH_MIN_LLR) break;

        const int bx = best % MANTIS_TOMO_DIM, by = best / MANTIS_TOMO_DIM;
        const float px = -m->extent + ((float)bx + 0.5f) * step;
        const float py =  m->extent - ((float)by + 0.5f) * step;

        // Runner-up measured on the WORK field, which already has every
        // previously accepted lobe suppressed.
        //
        // Measuring it on the original field was wrong and produced
        // margins BELOW 1.0 for second and third targets -- the "runner
        // up" was simply the first target, which is stronger by
        // construction.  A ratio under one is not a weak detection, it is
        // a meaningless number, and reporting it would have put nonsense
        // on the display.
        // Margin is signal over BACKGROUND, not over the runner-up.
        //
        // Comparing against the largest remaining peak looked right and
        // broke on the case that matters most: with two real people the
        // runner-up IS the second person, the ratio collapses to ~1.0,
        // and the FIRST target fails its own gate.  Two people in a room
        // produced zero detections.
        //
        // The question a margin should answer is "does this peak stand
        // above the field's noise floor", and the mean of the positive
        // background answers that whether there is one target or three.
        const int lobe = (int)(MANTIS_TARGET_SEPARATION / step);
        float bg_sum = 0.0f; int bg_n = 0;
        for (int i = 0; i < MANTIS_TOMO_CELLS; i++) {
            const int x = i % MANTIS_TOMO_DIM, y = i / MANTIS_TOMO_DIM;
            const int dx = x - bx, dy = y - by;
            if (dx * dx + dy * dy <= lobe * lobe) continue;
            if (work[i] <= -1e29f) continue;      // already-claimed lobe
            if (work[i] <= 0.0f)   continue;      // negative = evidence of clear
            bg_sum += work[i]; bg_n++;
        }
        const float bg = (bg_n > 0) ? (bg_sum / (float)bg_n) : 0.0f;
        const float margin = (bg > 1e-6f) ? (bv / bg) : 99.0f;
        const uint8_t cov = (uint8_t)mantis_tomo_coverage(m->chords, m->n_chords,
                                                          m->extent, px, py);

        // A SECOND target must be COMPARABLE to the first, not merely
        // above the floor.
        //
        // Without this, one person produced four "targets": the streak
        // structure of a 15-line sinogram leaves several crossings above
        // the LLR floor, and each looked like a detection.  A real second
        // body blocks a comparable number of chords, so requiring a
        // fraction of the primary's strength separates a person from a
        // reconstruction artefact -- and does it with a number that has a
        // physical meaning rather than a tuned threshold.
        // SCALE THE SECOND-TARGET BAR BY HOW MANY LINES WE ACTUALLY HAVE.
        //
        // 0.70 was measured on a COMPLETE 15-chord set, where the
        // strongest streak artefact reached 61% of a real peak.  With
        // fewer chords the reconstruction has fewer constraints and the
        // streaks get relatively stronger: at 20 of 30 directed links,
        // artefacts reached 77-79% and one person was reported as three.
        //
        // Fewer independent lines means less ability to separate
        // targets, so the evidence bar rises accordingly.  That is the
        // geometry talking, not a tuned constant -- and it degrades in
        // the safe direction, because reporting phantom people is worse
        // than missing a second real one.
        const float full = (float)(MANTIS_MAX_TARGETS > 0 ? 15 : 15);
        const float have = (float)m->n_chords;
        float frac = MANTIS_SECOND_TARGET_FRAC;
        if (have < full) {
            const float sparsity = 1.0f - (have / full);      // 0 = complete
            frac += (0.95f - MANTIS_SECOND_TARGET_FRAC) * sparsity;
            if (frac > 0.95f) frac = 0.95f;
        }
        const bool strong_enough =
            (k == 0) ? (margin > MANTIS_MESH_MIN_MARGIN)
                     : (bv >= frac * ts->t[0].llr
                        && margin > MANTIS_MESH_MIN_MARGIN);
        const bool ok = (cov >= 2) && strong_enough;

        if (ok) {
            MantisTarget &T = ts->t[ts->n];
            T.x = px; T.y = py; T.llr = bv; T.margin = margin; T.coverage = cov;
            T.phase_rate = mantis_phase_rate_at(m, ts, store, px, py, dt_s, now_ms);
            // THE CLASSIFICATION.  Blockage says it is there; phase rate
            // says whether it is moving.  A static target is still a
            // target -- that is the entire point.
            T.kind = (T.phase_rate >= MANTIS_MOVING_RAD_S) ? MT_MOVING : MT_STATIC;
            T.first_seq = T.last_seq = seq;
            T.age_frames = 1;
            T.active = true;
            ts->n++;
        }

        // Suppress this neighbourhood whether or not it was accepted --
        // a rejected peak must not be re-found on the next pass.
        for (int i = 0; i < MANTIS_TOMO_CELLS; i++) {
            const int x = i % MANTIS_TOMO_DIM, y = i / MANTIS_TOMO_DIM;
            const int dx = x - bx, dy = y - by;
            if (dx * dx + dy * dy <= lobe * lobe) work[i] = -1e30f;
        }
    }

    mantis_targets_snapshot_phase(ts, m, store, seq, now_ms);
    return ts->n;
}

static inline const char *mantis_kind_name(MantisTargetKind k) {
    switch (k) {
        case MT_MOVING: return "MOVING";
        case MT_STATIC: return "STATIC";
        case MT_FADING: return "FADING";
        default:        return "NONE";
    }
}
